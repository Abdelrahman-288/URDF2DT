"""Versioned executable reference polynomials and explicit controller samples."""

from dataclasses import asdict
from pathlib import Path
import numpy as np
from urdf2dt.config import EditorConfig
from urdf2dt.parser.urdf_input import URDFInput
from urdf2dt.parser.urdf_parser import SerialURDFParser
from urdf2dt.parser.urdf_validator import validate_urdf
from urdf2dt.identification.data import write_record, read_record, joint_units
from .core import MotionLimits, Trajectory, TrajectoryError


def export_trajectory(trajectory: Trajectory, source: URDFInput, path: str | Path, samples: int = 201) -> Path:
    chain = SerialURDFParser().parse(validate_urdf(source).require_valid(), EditorConfig())
    if (chain.source_sha256 != trajectory.chain.source_sha256 or chain.joints != trajectory.chain.joints
        or chain.base_link != trajectory.chain.base_link or chain.tip_link != trajectory.chain.tip_link):
        raise TrajectoryError("source_mismatch", "Export source differs from reference chain")
    return write_record(path, "urdf2dt.trajectory", {
        "source_name": source.name, "source_hex": source.content.hex(), "source_sha256": source.sha256,
        "joint_names": chain.joint_names, "position_units": joint_units(chain), "time_unit": "s",
        "derivative_units": "position_unit/s and position_unit/s^2", "pose_frame": chain.base_link,
        "times": trajectory.times, "coefficients": trajectory.coefficients,
        "coefficient_convention": "ascending powers of local u=(t-t_start)/segment_duration",
        "limits": asdict(trajectory.limits), "metadata": trajectory.metadata,
        "bounds": trajectory.bounds, "samples": trajectory.sample(samples), "requested_sample_count": samples})


def load_trajectory(path: str | Path) -> Trajectory:
    data = read_record(path, "urdf2dt.trajectory")
    source = URDFInput.from_upload(data["source_name"], bytes.fromhex(data["source_hex"]))
    chain = SerialURDFParser().parse(validate_urdf(source).require_valid(), EditorConfig())
    if (source.sha256 != data["source_sha256"] or list(chain.joint_names) != data["joint_names"]
        or list(joint_units(chain)) != data["position_units"] or data["time_unit"] != "s"
        or data["derivative_units"] != "position_unit/s and position_unit/s^2"
        or data["pose_frame"] != chain.base_link
        or data["coefficient_convention"] != "ascending powers of local u=(t-t_start)/segment_duration"):
        raise TrajectoryError("archive_mismatch", "Source, ordering, units or coefficient convention differs")
    trajectory = Trajectory(chain, data["times"], data["coefficients"], MotionLimits(**data["limits"]), data["metadata"])
    expected = trajectory.sample(data["requested_sample_count"])
    for key, values in expected.items():
        saved = np.asarray(data["samples"][key], dtype=float)
        array = np.asarray(values)
        if saved.shape != array.shape or not np.allclose(saved, array, atol=1e-10, rtol=1e-10):
            raise TrajectoryError("archive_mismatch", f"Stored {key} differs from analytic reference")
    return trajectory
