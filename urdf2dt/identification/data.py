"""Owned, versioned SI time series; trajectory identifiers define split boundaries."""

from dataclasses import dataclass, asdict
from hashlib import sha256
import json
from pathlib import Path
from typing import Any
import numpy as np

from urdf2dt.dh.types import KinematicChain, JointType


def canonical(value: Any) -> bytes:
    return json.dumps(value, sort_keys=True, separators=(",", ":"), allow_nan=False).encode()


def write_record(path: str | Path, kind: str, data: dict) -> Path:
    payload = {"schema_version": "1.0", "kind": kind, **data}
    envelope = {"payload": payload, "sha256": sha256(canonical(payload)).hexdigest()}
    target = Path(path)
    with target.open("x", encoding="utf-8") as stream:
        json.dump(envelope, stream, indent=2, allow_nan=False)
        stream.write("\n")
    return target


def read_record(path: str | Path, kind: str) -> dict:
    target = Path(path)
    if target.stat().st_size > 64*1024*1024:
        raise ValueError("Identification record exceeds 64 MiB")
    envelope = json.loads(target.read_text(encoding="utf-8"))
    data = envelope["payload"]
    if sha256(canonical(data)).hexdigest() != envelope["sha256"]:
        raise ValueError("Identification record checksum mismatch")
    if data.pop("kind") != kind or data.pop("schema_version") != "1.0":
        raise ValueError("Unsupported identification record")
    return data


def joint_units(chain: KinematicChain) -> tuple[str, ...]:
    return tuple("m" if j.joint_type == JointType.PRISMATIC else "rad"
                 for j in chain.joints if j.joint_type != JointType.FIXED)


@dataclass(frozen=True)
class CurrentCalibration:
    """Measured affine current-to-joint-effort calibration, including transmission.

    Gains are signed N*m/A or N/A, offsets are amperes. These are supplied,
    independently calibrated values, never inferred together with unknown masses.
    """

    gain: tuple[float, ...]
    offset_ampere: tuple[float, ...]
    effort_units: tuple[str, ...]
    provenance: str

    def __post_init__(self) -> None:
        for key in ("gain", "offset_ampere", "effort_units"):
            object.__setattr__(self, key, tuple(getattr(self, key)))
        if (not self.gain or len(self.gain) != len(self.offset_ampere)
            or len(self.gain) != len(self.effort_units) or not self.provenance.strip()
            or not np.isfinite(self.gain + self.offset_ampere).all()
            or any(g == 0 for g in self.gain)
            or any(u not in {"N*m", "N"} for u in self.effort_units)):
            raise ValueError("Supply finite nonzero calibrated gains, offsets, SI units and provenance")

    def convert(self, current: Any, position_units: tuple[str, ...]) -> np.ndarray:
        expected = tuple("N" if u == "m" else "N*m" for u in position_units)
        array = np.asarray(current, dtype=float)
        if (expected != self.effort_units or array.ndim != 2
            or array.shape[1] != len(self.gain) or not np.isfinite(array).all()):
            raise ValueError("Current dimensions/units must match the calibrated joint order")
        return (array - np.asarray(self.offset_ampere)) * np.asarray(self.gain)


@dataclass(frozen=True)
class IdentificationData:
    """One or more complete trajectories, supplied derivatives, joint-side effort.

    processing must describe differentiation/filtering, synchronization and noise
    assumptions. No implicit derivative estimation or filtering is performed.
    q/velocity/acceleration use position_units, position_units/s, position_units/s².
    Effort is N*m for rad and N for m; timestamps are seconds.
    """

    source_sha256: str
    joint_names: tuple[str, ...]
    position_units: tuple[str, ...]
    trajectory: tuple[str, ...]
    time: tuple[float, ...]
    q: tuple[tuple[float, ...], ...]
    velocity: tuple[tuple[float, ...], ...]
    acceleration: tuple[tuple[float, ...], ...]
    effort: tuple[tuple[float, ...], ...]
    origin: str
    processing: str
    acquisition_id: str
    current_calibration: CurrentCalibration | None = None
    current_ampere: tuple[tuple[float, ...], ...] | None = None

    def __post_init__(self) -> None:
        for key in ("joint_names", "position_units", "trajectory", "time"):
            object.__setattr__(self, key, tuple(getattr(self, key)))
        n, count = len(self.joint_names), len(self.time)
        if (not n or len(set(self.joint_names)) != n or any(not s for s in self.joint_names)
            or len(self.position_units) != n or any(u not in {"m", "rad"} for u in self.position_units)
            or count < 2 or len(self.trajectory) != count or any(not s for s in self.trajectory)
            or not np.isfinite(self.time).all()
            or self.origin not in {"simulated", "measured"}
            or not self.processing.strip() or not self.acquisition_id.strip()
            or len(self.source_sha256) != 64
            or any(c not in "0123456789abcdef" for c in self.source_sha256)):
            raise ValueError("Invalid dataset identity, units, timestamps or processing metadata")
        for name in set(self.trajectory):
            times = np.asarray(self.time)[np.array(self.trajectory) == name]
            if len(times) < 2 or np.any(np.diff(times) <= 0):
                raise ValueError("Timestamps must strictly increase within each trajectory")
        for key in ("q", "velocity", "acceleration", "effort", "current_ampere"):
            value = getattr(self, key)
            if value is None and key == "current_ampere":
                continue
            array = np.asarray(value, dtype=float)
            if array.shape != (count, n) or not np.isfinite(array).all():
                raise ValueError(f"{key} must be a finite samples-by-joints matrix")
            object.__setattr__(self, key, tuple(tuple(float(v) for v in row) for row in array))
        if (self.current_calibration is None) != (self.current_ampere is None):
            raise ValueError("Current input requires retained raw current and calibrated conversion")
        if self.current_calibration is not None:
            converted = self.current_calibration.convert(self.current_ampere, self.position_units)
            if not np.allclose(converted, self.effort, rtol=1e-12, atol=1e-12):
                raise ValueError("Effort does not match retained current calibration")

    def validate_chain(self, chain: KinematicChain) -> None:
        if (self.source_sha256 != chain.source_sha256 or self.joint_names != chain.joint_names
            or self.position_units != joint_units(chain)):
            raise ValueError("Dataset source, joint order or SI units differ from the chain")

    @property
    def digest(self) -> str:
        return sha256(canonical(asdict(self))).hexdigest()

    def save(self, path: str | Path) -> Path:
        return write_record(path, "urdf2dt.identification_data", asdict(self))

    @classmethod
    def load(cls, path: str | Path) -> "IdentificationData":
        data = read_record(path, "urdf2dt.identification_data")
        if data.get("current_calibration") is not None:
            data["current_calibration"] = CurrentCalibration(**data["current_calibration"])
        return cls(**data)


def require_held_out(training: IdentificationData, evaluation: IdentificationData) -> None:
    if (training.source_sha256 != evaluation.source_sha256
        or training.joint_names != evaluation.joint_names
        or training.position_units != evaluation.position_units):
        raise ValueError("Training and evaluation must describe the same chain")
    if set(training.trajectory) & set(evaluation.trajectory):
        raise ValueError("Hold out entire trajectories: identifiers overlap")
    # Catch relabelled exact samples too. This is a guard, not proof of independence.
    train_states = set(zip(training.q, training.velocity, training.acceleration))
    if any(state in train_states for state in zip(evaluation.q, evaluation.velocity, evaluation.acceleration)):
        raise ValueError("Training and evaluation contain identical states")
