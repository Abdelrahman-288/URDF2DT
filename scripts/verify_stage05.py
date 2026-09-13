"""Reproducible Stage 5 UR5 development check, independent of production FK math.

Run from the repository: python scripts/verify_stage05.py --output <report.json>.
This is development evidence, not the Stage 11 global validator or Stage 12 export.
"""

import argparse
from dataclasses import asdict
from datetime import datetime, timezone
import json
from math import pi
from pathlib import Path
import platform
import subprocess

from defusedxml.ElementTree import fromstring
import numpy as np
import scipy
from scipy.spatial.transform import Rotation

from urdf2dt.pipeline import generate_automatic_model
from urdf2dt.kinematics import dh_fk, urdf_fk


def reference_urdf_fk(document, q):
    """Independent XML/SciPy FK: does not use parser rotation or FK helpers."""
    root = fromstring(document.source.content, forbid_dtd=True, forbid_entities=True, forbid_external=True)
    raw = {joint.attrib["name"]: joint for joint in root.findall("joint")}
    pose = np.eye(4)
    values = iter(q)
    for summary in document.joints:
        joint = raw[summary.name]
        origin = joint.find("origin")
        fixed = np.eye(4)
        if origin is not None:
            fixed[:3, 3] = [float(v) for v in origin.get("xyz", "0 0 0").split()]
            fixed[:3, :3] = Rotation.from_euler("xyz", [float(v) for v in origin.get("rpy", "0 0 0").split()]).as_matrix()
        pose = pose @ fixed
        if joint.attrib["type"] != "fixed":
            axis_element = joint.find("axis")
            axis = np.array([float(v) for v in (axis_element.get("xyz") if axis_element is not None else "1 0 0").split()])
            axis /= np.linalg.norm(axis)
            value = next(values)
            motion = np.eye(4)
            if joint.attrib["type"] == "prismatic":
                motion[:3, 3] = value * axis
            else:
                motion[:3, :3] = Rotation.from_rotvec(value * axis).as_matrix()
            pose = pose @ motion
    return pose


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    root = Path(__file__).resolve().parents[1]
    commit = subprocess.check_output(["git", "rev-parse", "HEAD"], cwd=root, text=True).strip()
    dirty = subprocess.check_output(["git", "status", "--porcelain"], cwd=root, text=True).strip()
    run = generate_automatic_model(root / "robots/ur5/ur5_serial.urdf")
    config = run.config
    rng = np.random.default_rng(config.validation.random_seed)
    movable = [j for j in run.chain.joints if j.joint_type.value != "fixed"]
    configurations = [np.zeros(len(movable))]
    for _ in range(config.validation.samples - 1):
        configurations.append(np.array([rng.uniform(j.limit.lower, j.limit.upper) for j in movable]))
    samples = []
    for q in configurations:
        reference = reference_urdf_fk(run.source, q)
        dh = np.array(dh_fk(run.automatic_model, q))
        urdf = np.array(urdf_fk(run.chain, q))
        samples.append({
            "q": q.tolist(),
            "position_error_m": float(np.linalg.norm(dh[:3, 3] - reference[:3, 3])),
            "orientation_error_rad": float(Rotation.from_matrix(reference[:3, :3].T @ dh[:3, :3]).magnitude()),
            "urdf_evaluator_max_abs_error": float(np.max(np.abs(urdf - reference))),
        })
    # Manufacturer's classic UR5 nominal table, independently transcribed.
    # Source and frame convention are documented in docs/stages/05_parser_integration.md.
    reference_table = np.array([[0, pi/2, .089159, 0], [-.425, 0, 0, 0],
                                [-.39225, 0, 0, 0], [0, pi/2, .10915, 0],
                                [0, -pi/2, .09465, 0], [0, 0, .0823, 0]])
    automatic_table = np.array([[r.a, r.alpha, r.d, r.theta_offset] for r in run.automatic_model.rows])
    errors = np.max(np.abs(automatic_table - reference_table), axis=0)
    max_position = max(s["position_error_m"] for s in samples)
    max_orientation = max(s["orientation_error_rad"] for s in samples)
    passed = (max_position <= config.validation.position_tolerance_m
              and max_orientation <= config.validation.orientation_tolerance_rad
              and bool(np.all(errors <= 1.0e-4)))
    report = {
        "report_kind": "stage05-development-check", "generated_at": datetime.now(timezone.utc).isoformat(),
        "implementation_commit": commit, "working_tree_clean_at_start": not bool(dirty),
        "python": platform.python_version(), "numpy": np.__version__, "scipy": scipy.__version__,
        "source_file": "robots/ur5/ur5_serial.urdf", "source_sha256": run.chain.source_sha256,
        "configuration": config.snapshot(), "joint_names": run.chain.joint_names,
        "base_transform": run.automatic_model.base_transform, "tool_transform": run.automatic_model.tool_transform,
        "dh_rows": [asdict(row) for row in run.automatic_model.rows],
        "reference_table_max_errors": dict(zip(("a_m", "alpha_rad", "d_m", "theta_offset_rad"), errors.tolist())),
        "reference_table_tolerance": 1.0e-4,
        "reference_table_source": "https://www.universal-robots.com/articles/ur/application-installation/dh-parameters-for-calculations-of-kinematics-and-dynamics",
        "max_position_error_m": max_position, "max_orientation_error_rad": max_orientation,
        "passed_provisional_checks": passed, "samples": samples,
        "limitations": ["Prepared public serial fixture, not the missing MATLAB universalUR5.urdf",
                        "Advisor thresholds and MATLAB comparison remain unconfirmed",
                        "No editor/global-validator certification is implied"],
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2, allow_nan=False) + "\n", encoding="utf-8")
    print(f"{len(samples)} UR5 samples; position max={max_position:.3g} m; orientation max={max_orientation:.3g} rad")
    print(f"Reference table max errors [a, alpha, d, theta]: {errors}")
    print(f"Provisional checks: {'PASS' if passed else 'FAIL'}; report: {args.output}")
    return 0 if passed else 1


if __name__ == "__main__":
    raise SystemExit(main())
