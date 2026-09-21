"""Controlled threshold, legal-edit, sample-density, and boundary experiments."""

from dataclasses import asdict, replace
from hashlib import sha256
from math import pi, sqrt
from pathlib import Path
import csv
import json
from typing import Any

from urdf2dt.config import EditorConfig, GeometryConfig
from urdf2dt.dh.classification import (
    classify_axis_pair,
    classify_dh_model,
    get_editable_params,
)
from urdf2dt.dh.editor_session import EditorSession
from urdf2dt.dh.global_validation import (
    model_hash,
    sample_configurations,
    validate_global_fk,
)
from urdf2dt.dh.recompute import FrameEdit, validate_frame
from urdf2dt.dh.types import DHModel
from urdf2dt.kinematics import dh_frame_transforms
from urdf2dt.logging_config import git_provenance
from urdf2dt.parser.urdf_input import URDFInput
from urdf2dt.pipeline import AutomaticDHResult, generate_automatic_model


def _classification(case: Any) -> dict[str, Any]:
    return {
        **asdict(case),
        "case": case.case.value,
        "editable_parameters": [p.name for p in get_editable_params(case)],
    }


def boundary_study(config: GeometryConfig) -> list[dict[str, Any]]:
    """Probe both sides of each threshold, retaining measured rather than assumed geometry."""
    records = []
    for factor in (0.0, 0.1, 0.5, 0.999, 1.0, 1.001, 2.0, 10.0):
        sine = config.parallel_threshold * factor
        if sine >= 1:
            continue
        specifications = (
            (
                "parallel",
                (0.2, 0.0, 0.0),
                (sine, 0.0, sqrt(1 - sine * sine)),
                config.parallel_threshold,
            ),
            (
                "intersection",
                (0.0, factor * config.intersection_threshold, 0.0),
                (1.0, 0.0, 0.0),
                config.intersection_threshold,
            ),
            (
                "coincidence",
                (factor * config.common_normal_threshold, 0.0, 0.0),
                (0.0, 0.0, 1.0),
                config.common_normal_threshold,
            ),
        )
        for family, origin, direction, threshold in specifications:
            case = classify_axis_pair(
                (0.0, 0.0, 0.0), (0.0, 0.0, 1.0), origin, direction, config
            )
            records.append(
                {
                    "family": family,
                    "factor": factor,
                    "threshold": threshold,
                    "origin_a": (0.0, 0.0, 0.0),
                    "direction_a": (0.0, 0.0, 1.0),
                    "origin_b": origin,
                    "direction_b": direction,
                    **_classification(case),
                }
            )
    return records


def _accepted_candidate(run: AutomaticDHResult, edits: dict[int, FrameEdit]) -> DHModel:
    """Apply edits through the real sequential acceptance gate, starting fresh each time."""
    session = EditorSession(run.automatic_model, geometry=run.config.geometry)
    for index in range(1, len(run.automatic_model.rows) + 1):
        session.unlock_next()
        session.propose_edit(index, edits.get(index, FrameEdit()))
        decision = session.accept()
        if not decision.valid:
            raise ValueError(decision.reason)
    if session.state.automatic_model != run.automatic_model:
        raise AssertionError("Experiment mutated the automatic baseline")
    return session.state.working_model


def _metrics(
    run: AutomaticDHResult, model: DHModel, config: EditorConfig
) -> dict[str, Any]:
    report = validate_global_fk(run.chain, model, config).to_dict()
    return {
        "passed": report["passed"],
        "candidate_hash": report["candidate_hash"],
        **report["summary"],
        "samples": config.validation.samples,
        "diagnostic": report["diagnostic"],
        "sample_errors": [
            [s["position_error_m"], s["orientation_error_rad"]]
            for s in report["samples"]
        ],
    }


def run_studies(
    source: str | Path | URDFInput, *, sweep_points: int = 21, seed: int = 42
) -> dict[str, Any]:
    """Run finite, independent one-parameter sweeps; never export research candidates.

    Translation is mathematically unbounded: [-1, 1] m is the tested interval.
    Rotation spans one period [-pi, pi]. All finite grids include both ends and zero.
    Failures/rejections stay in the record; unexpected errors propagate to the caller.
    """
    if type(sweep_points) is not int or sweep_points < 3 or sweep_points % 2 == 0:
        raise ValueError("sweep_points must be an odd integer of at least three")
    config = EditorConfig()
    config = replace(config, validation=replace(config.validation, random_seed=seed))
    run = generate_automatic_model(source, config)
    baseline = run.automatic_model
    original_hash = model_hash(baseline)
    baseline_metrics = _metrics(run, baseline, config)
    if not baseline_metrics["passed"]:
        raise ValueError("Automatic baseline failed sampled FK; studies cancelled")
    classifications = classify_dh_model(baseline, config.geometry)
    thresholds = []
    for factor in (0.1, 0.3, 1.0, 3.0, 10.0):
        geometry = replace(
            config.geometry,
            parallel_threshold=config.geometry.parallel_threshold * factor,
        )
        cases = classify_dh_model(baseline, geometry)
        thresholds.append(
            {
                "factor": factor,
                "configuration": asdict(geometry),
                "changed_frames": [
                    i
                    for i, (a, b) in enumerate(zip(classifications, cases), 1)
                    if (a.case, a.coincident, a.requires_review)
                    != (b.case, b.coincident, b.requires_review)
                ],
                "frames": [_classification(c) for c in cases],
            }
        )
    if not any(get_editable_params(case) for case in classifications):
        raise ValueError("This robot has no continuous legal frame freedoms to sweep")
    sweep = []
    zero_pose = (0.0,) * len(baseline.rows)
    automatic_frames = dh_frame_transforms(baseline, zero_pose)
    combined_edits: dict[int, FrameEdit] = {}
    for index, case in enumerate(classifications, 1):
        for parameter in get_editable_params(case):
            extent = 1.0 if parameter.name == "axial_translation" else pi
            for point in range(sweep_points):
                value = extent * (2 * point / (sweep_points - 1) - 1)
                edit = FrameEdit(axial_translation=value) if parameter.name=="axial_translation" else FrameEdit(axial_rotation=value)
                record = {
                    "frame": index,
                    "parameter": parameter.name,
                    "unit": parameter.unit,
                    "value": value,
                    "interval": [-extent, extent],
                    "points": sweep_points,
                }
                try:
                    candidate = _accepted_candidate(run, {index: edit})
                    frames = dh_frame_transforms(candidate, zero_pose)
                    local = validate_frame(
                        automatic_frames[index - 1],
                        automatic_frames[index],
                        frames[index],
                        config.geometry,
                    )
                    record.update(
                        {
                            "outcome": "evaluated",
                            "local_valid": local.valid,
                            "local_issues": [asdict(issue) for issue in local.issues],
                            **_metrics(run, candidate, config),
                        }
                    )
                except ValueError as exc:
                    record.update(
                        {
                            "outcome": "rejected",
                            "local_valid": False,
                            "passed": False,
                            "reason": str(exc),
                        }
                    )
                sweep.append(record)
            previous = combined_edits.get(index, FrameEdit())
            combined_edits[index] = (replace(previous,axial_translation=.05) if parameter.name=="axial_translation"
                                     else replace(previous,axial_rotation=.2))
    edited = _accepted_candidate(run, combined_edits)
    # Deliberate corruption checks detection. It bypasses acceptance only inside this experiment.
    rows = list(baseline.rows)
    fault_index = min(2, len(rows) - 1)
    rows[fault_index] = replace(rows[fault_index], a=rows[fault_index].a + 0.001)
    negative = replace(baseline, rows=tuple(rows))
    density = []
    samples = {}
    for count in (10, 50, 200):
        setting = replace(config, validation=replace(config.validation, samples=count))
        samples[str(count)] = sample_configurations(run.chain, setting)
        for name, model in (
            ("automatic", baseline),
            ("edited", edited),
            ("negative_control", negative),
        ):
            density.append(
                {
                    "model": name,
                    "expected_pass": name != "negative_control",
                    **_metrics(run, model, setting),
                }
            )
    if model_hash(baseline) != original_hash:
        raise AssertionError("Automatic baseline changed during studies")
    return {
        "study_schema": "urdf2dt-ablation-v1",
        "provenance": git_provenance(),
        "source": {
            "name": run.source.source.name,
            "sha256": run.chain.source_sha256,
            "path": run.chain.source_urdf,
            "base": run.chain.base_link,
            "tip": run.chain.tip_link,
            "joint_names": run.chain.joint_names,
        },
        "configuration": config.snapshot(),
        "baseline": asdict(baseline),
        "baseline_metrics": baseline_metrics,
        "method": {
            "convention": "Standard DH including base/tool transforms",
            "threshold_scope": "Reclassify the SAME automatic model; no solver change or FK-tolerance change",
            "sweep": "Independent fresh session for every parameter/value; other freedoms zero",
            "translation_domain": "Unbounded; only [-1,1] m sampled, not exhaustive",
            "rotation_domain": "One period [-pi,pi] sampled, not continuous coverage",
            "sample_order": "Python Random(seed), zero first, nested prefixes across sample counts",
            "errors": "sample_errors columns: position_m, orientation_rad; order matches sample_sets",
            "rule_names": "Rigidity, directed joint line and common normal; report R1-R3 mapping provisional",
        },
        "thresholds": thresholds,
        "sweep": sweep,
        "density": density,
        "sample_sets": samples,
        "boundary": boundary_study(config.geometry),
        "edited_model": asdict(edited),
        "combined_edits": {str(i): asdict(e) for i, e in combined_edits.items()},
        "negative_control": {
            "frame": fault_index + 1,
            "change": "a += 0.001 m",
            "candidate": asdict(negative),
        },
        "expected_outcomes_met": (
            bool(sweep)
            and all(r["local_valid"] and r["passed"] for r in sweep)
            and all(r["passed"] == r["expected_pass"] for r in density)
        ),
    }


def write_tables(result: dict[str, Any], folder: Path) -> None:
    """Write human-readable tables and the full reproducibility record into a new folder."""
    folder.mkdir(parents=True, exist_ok=False)
    (folder / "results.json").write_text(
        json.dumps(result, indent=2, allow_nan=False) + "\n", encoding="utf-8"
    )
    for name, fields in (
        (
            "sweep",
            [
                "frame",
                "parameter",
                "unit",
                "value",
                "outcome",
                "local_valid",
                "passed",
                "max_position_error_m",
                "max_orientation_error_rad",
                "mean_position_error_m",
                "mean_orientation_error_rad",
                "reason",
            ],
        ),
        (
            "density",
            [
                "model",
                "samples",
                "expected_pass",
                "passed",
                "max_position_error_m",
                "max_orientation_error_rad",
                "mean_position_error_m",
                "mean_orientation_error_rad",
                "worst_position_sample",
                "worst_orientation_sample",
            ],
        ),
        (
            "boundary",
            [
                "family",
                "factor",
                "threshold",
                "sine_angle",
                "distance_m",
                "case",
                "coincident",
                "requires_review",
                "editable_parameters",
            ],
        ),
    ):
        with (folder / f"{name}.csv").open("w", newline="", encoding="utf-8") as stream:
            writer = csv.DictWriter(stream, fieldnames=fields, extrasaction="ignore")
            writer.writeheader()
            writer.writerows(result[name])


def code_fingerprint(root: Path) -> dict[str, str]:
    """Hash research/core source files so unrelated dirty notebooks cannot obscure code identity."""
    paths = sorted((root / "urdf2dt").rglob("*.py")) + [root / "scripts/run_stage14.py"]
    return {
        p.relative_to(root).as_posix(): sha256(p.read_bytes()).hexdigest()
        for p in paths
    }
