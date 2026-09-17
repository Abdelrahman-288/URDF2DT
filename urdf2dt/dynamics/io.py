"""Versioned dynamics archives retain exact source bytes and independently parsed parameters."""

from dataclasses import asdict
from hashlib import sha256
import json
from pathlib import Path
from typing import Any

from urdf2dt.dynamics.model import DynamicModel, DynamicsConfig, JointFriction
from urdf2dt.parser.inertial_extractor import extract_inertials
from urdf2dt.parser.urdf_input import URDFInput
from urdf2dt.parser.xml_snapshot import read_snapshot
from urdf2dt.pipeline import generate_automatic_model
from urdf2dt.dh.global_validation import validate_global_fk
from urdf2dt.config import EditorConfig
from urdf2dt.parser.urdf_parser import SerialURDFParser
from urdf2dt.parser.urdf_validator import validate_urdf

UNITS = {"length": "m", "mass": "kg", "inertia": "kg*m^2",
         "angle": "rad", "revolute_effort": "N*m", "prismatic_effort": "N"}


def load_dynamic_model(source: str | Path | URDFInput, config: DynamicsConfig | None = None) -> DynamicModel:
    """Validate source kinematics/FK, extract independent inertias, then compile dynamics.

    With no explicit config, URDF damping/friction is used; absent coefficients
    mean a documented frictionless assumption. Explicit config replaces that policy.
    """
    run = generate_automatic_model(source)
    inertials = extract_inertials(run.source.source)
    if not validate_global_fk(run.chain, run.automatic_model, run.config).passed:
        raise ValueError("Kinematic FK validation failed")
    if config is None:
        friction = []
        root = read_snapshot(run.source.source)
        for joint in root.findall("joint"):
            if joint.get("name") not in run.chain.joint_names:
                continue
            entries = joint.findall("dynamics")
            if len(entries) > 1:
                raise ValueError("Multiple joint dynamics elements")
            if entries:
                friction.append(JointFriction(joint.attrib["name"],
                    float(entries[0].get("damping", "0")), float(entries[0].get("friction", "0"))))
        config = DynamicsConfig(friction=tuple(friction))
    return DynamicModel(run.chain, inertials, config)


def _canonical(data: Any) -> bytes:
    return json.dumps(data, sort_keys=True, separators=(",", ":"), allow_nan=False).encode()


def export_model(model: DynamicModel, source: URDFInput, path: str | Path) -> Path:
    """Write a new JSON file; refuse stale or edited source/parameter associations."""
    if source.sha256 != model.chain.source_sha256 or extract_inertials(source) != model.inertials:
        raise ValueError("Export source does not match model inertias")
    chain = SerialURDFParser().parse(validate_urdf(source).require_valid(), EditorConfig())
    if (chain.joints != model.chain.joints or chain.base_link != model.chain.base_link
        or chain.tip_link != model.chain.tip_link or chain.robot_name != model.chain.robot_name):
        raise ValueError("Export chain differs from the source URDF")
    data = {"schema_version": "1.0", "kind": "urdf2dt.dynamic_model",
            "source_name": source.name, "source_hex": source.content.hex(),
            "source_sha256": source.sha256, "joint_names": model.chain.joint_names,
            "inertials": asdict(model.inertials), "config": asdict(model.config),
            "units": UNITS,
            "assumptions": ["fixed base", "rigid serial chain", "no external contact/wrenches",
                            "no transmission or rotor inertia", "Coulomb sign(0)=0; no stiction",
                            "absent friction coefficients assumed zero", "SI units required; URDF has no unit tags"]}
    envelope = {"payload": data, "sha256": sha256(_canonical(data)).hexdigest()}
    destination = Path(path)
    with destination.open("x", encoding="utf-8") as stream:
        json.dump(envelope, stream, indent=2, allow_nan=False)
        stream.write("\n")
    return destination


def load_model_archive(path: str | Path) -> DynamicModel:
    """Rebuild numeric expressions after integrity, source and parameter checks."""
    target = Path(path)
    if target.stat().st_size > 25 * 1024 * 1024:
        raise ValueError("Dynamics archive exceeds size limit")
    envelope = json.loads(target.read_text(encoding="utf-8"))
    data = envelope["payload"]
    if envelope["sha256"] != sha256(_canonical(data)).hexdigest():
        raise ValueError("Dynamics archive checksum mismatch")
    if data["schema_version"] != "1.0" or data["kind"] != "urdf2dt.dynamic_model":
        raise ValueError("Unsupported dynamics archive")
    if data["units"] != UNITS:
        raise ValueError("Dynamics archive must use documented SI units")
    source = URDFInput.from_upload(data["source_name"], bytes.fromhex(data["source_hex"]))
    if source.sha256 != data["source_sha256"]:
        raise ValueError("Source checksum mismatch")
    settings = data["config"]
    config = DynamicsConfig(tuple(settings["gravity"]),
        tuple(JointFriction(**item) for item in settings["friction"]), settings["parameter_provenance"])
    model = load_dynamic_model(source, config)
    if list(model.chain.joint_names) != data["joint_names"] or _canonical(asdict(model.inertials)) != _canonical(data["inertials"]):
        raise ValueError("Archived joint order or inertias differ from source")
    return model
