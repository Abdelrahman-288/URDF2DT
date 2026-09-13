"""Verify that the draft schema is valid and preserves provisional provenance."""

from dataclasses import asdict
import json
from pathlib import Path

from jsonschema import Draft202012Validator
import pytest

from urdf2dt.config import EditorConfig
from tests.fixtures.ur5_automatic_dh import make_ur5_fixture


@pytest.fixture
def validator():
    path = Path(__file__).resolve().parents[1] / "schemas/dh-model-0.1.schema.json"
    schema = json.loads(path.read_text(encoding="utf-8"))
    Draft202012Validator.check_schema(schema)
    return Draft202012Validator(schema)


@pytest.fixture
def envelope():
    model = make_ur5_fixture()
    document = {
        "schema_version": "0.1", "dh_convention": "standard",
        "robot": {"name": model.robot_name, "source_urdf": model.source_urdf},
        "provenance": model.provenance, "is_temporary_fixture": model.is_temporary_fixture,
        "base_transform": model.base_transform, "tool_transform": model.tool_transform,
        "dh_parameters": [dict(index=i, **asdict(row)) for i, row in enumerate(model.rows, 1)],
        "configuration": EditorConfig().snapshot(),
    }
    return json.loads(json.dumps(document, allow_nan=False))


def test_nominal_fixture_envelope(validator, envelope):
    validator.validate(envelope)


@pytest.mark.parametrize("field,value", [
    ("schema_version", "1.0"), ("dh_convention", "modified"),
    ("is_temporary_fixture", "true"), ("dh_parameters", []),
    ("base_transform", [[1]]), ("passed", True),
])
def test_invalid_envelopes(validator, envelope, field, value):
    envelope[field] = value
    assert list(validator.iter_errors(envelope))


def test_configuration_and_row_contract(validator, envelope):
    envelope["configuration"]["validation"]["samples"] = 0
    envelope["dh_parameters"][0]["joint_type"] = "fixed"
    assert len(list(validator.iter_errors(envelope))) == 2
