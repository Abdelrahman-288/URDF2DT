"""Check experimental controls, failure detection, and deterministic grids."""

from pathlib import Path

import pytest

from urdf2dt.config import GeometryConfig
from urdf2dt.research.ablations import boundary_study, run_studies, write_tables

SOURCE = Path(__file__).resolve().parents[1] / "robots/ur5/ur5_serial.urdf"


@pytest.fixture(scope="module")
def study():
    return run_studies(SOURCE, sweep_points=3)


def test_boundaries_lock_approximate_parallel_cases():
    rows = boundary_study(GeometryConfig())
    assert rows == boundary_study(GeometryConfig())
    by_key = {(r["family"], r["factor"]): r for r in rows}
    below = by_key["parallel", 0.999]
    above = by_key["parallel", 1.001]
    assert below["case"] == "parallel" and below["requires_review"]
    assert not below["editable_parameters"]
    assert above["case"] == "intersecting"
    assert by_key["intersection", 0.999]["case"] == "intersecting"
    assert by_key["intersection", 1.001]["case"] == "skew"
    assert by_key["coincidence", 0.999]["coincident"]
    assert not by_key["coincidence", 1.001]["coincident"]


def test_studies_exercise_all_ur5_freedoms_and_detect_fault(study):
    assert study["expected_outcomes_met"]
    assert len(study["sweep"]) == 12
    groups = {(r["frame"], r["parameter"]) for r in study["sweep"]}
    assert groups == {
        (2, "axial_translation"),
        (3, "axial_translation"),
        (6, "axial_translation"),
        (6, "axial_rotation"),
    }
    for group in groups:
        rows = [r for r in study["sweep"] if (r["frame"], r["parameter"]) == group]
        assert rows[0]["value"] == rows[0]["interval"][0]
        assert rows[1]["value"] == 0
        assert rows[2]["value"] == rows[0]["interval"][1]
        assert all(r["local_valid"] and r["passed"] for r in rows)
        assert all(len(r["sample_errors"]) == 50 for r in rows)
        assert rows[1]["candidate_hash"] == study["baseline_metrics"]["candidate_hash"]
    assert all(not r["changed_frames"] for r in study["thresholds"])
    negatives = [r for r in study["density"] if r["model"] == "negative_control"]
    assert len(negatives) == 3
    assert all(not r["passed"] and r["max_position_error_m"] > 1e-4 for r in negatives)


def test_sample_sets_are_nested_and_metrics_match(study):
    assert study["sample_sets"]["10"] == study["sample_sets"]["200"][:10]
    assert study["sample_sets"]["50"] == study["sample_sets"]["200"][:50]
    for row in study["density"]:
        assert len(row["sample_errors"]) == row["samples"]
        assert row["max_position_error_m"] == max(p for p, a in row["sample_errors"])
        assert row["max_orientation_error_rad"] == max(
            a for p, a in row["sample_errors"]
        )


def test_existing_results_are_preserved(tmp_path, study):
    output = tmp_path / "results"
    write_tables(study, output)
    original = (output / "results.json").read_bytes()
    with pytest.raises(FileExistsError):
        write_tables(study, output)
    assert (output / "results.json").read_bytes() == original


@pytest.mark.parametrize("points", [0, 2, 4])
def test_grid_requires_endpoints_and_zero(points):
    with pytest.raises(ValueError, match="odd integer"):
        run_studies(SOURCE, sweep_points=points)


def test_cli_reporting_works_with_windows_legacy_encoding(tmp_path, study, monkeypatch):
    from copy import deepcopy
    import io
    import sys
    from scripts import run_stage14

    stream = io.TextIOWrapper(io.BytesIO(), encoding="cp1252")
    output = tmp_path / "cli"
    monkeypatch.setattr(sys, "stdout", stream)
    monkeypatch.setattr(
        sys, "argv", ["run_stage14.py", str(SOURCE), "--output", str(output)]
    )
    monkeypatch.setattr(run_stage14, "run_studies", lambda source: deepcopy(study))
    monkeypatch.setattr(run_stage14, "plot_studies", lambda result, folder: None)
    assert run_stage14.main() == 0
    stream.flush()
    assert b"Expected outcomes met: True" in stream.buffer.getvalue()
    assert "→" in (output / "summary.md").read_text(encoding="utf-8")
