"""Configuration safety, reproducibility and immutability contracts."""

from dataclasses import FrozenInstanceError
import json
from pathlib import Path

import pytest
import yaml

from urdf2dt.config import (
    ConfigError, EditorConfig, GeometryConfig, LoggingConfig, UIConfig,
    ValidationConfig, config_from_mapping, load_config,
)

ROOT = Path(__file__).resolve().parents[1]


def test_yaml_defaults_and_independent_snapshot():
    config = load_config(ROOT / "config/default.yaml")
    assert config == load_config()
    assert config.reference_status == "provisional"
    snapshot = json.loads(json.dumps(config.snapshot()))
    assert config_from_mapping(snapshot) == config
    snapshot["validation"]["samples"] = 4
    assert config.validation.samples == 50
    with pytest.raises(FrozenInstanceError):
        config.validation.samples = 4


@pytest.mark.parametrize("value", [0, -1, float("nan"), float("inf"), True, "1e-5", None])
def test_invalid_tolerances(value):
    with pytest.raises(ConfigError):
        GeometryConfig(parallel_threshold=value)
    with pytest.raises(ConfigError):
        ValidationConfig(position_tolerance_m=value)


@pytest.mark.parametrize("kwargs", [
    {"samples": 0}, {"samples": 2.5}, {"samples": True},
    {"random_seed": -1}, {"random_seed": False}, {"random_seed": "42"},
])
def test_sampling_contract(kwargs):
    with pytest.raises(ConfigError):
        ValidationConfig(**kwargs)


def test_other_configuration_constraints():
    with pytest.raises(ConfigError):
        GeometryConfig(parallel_threshold=1)
    with pytest.raises(ConfigError):
        UIConfig(live_preview="false")
    with pytest.raises(ConfigError):
        LoggingConfig(level="TRACE")
    with pytest.raises(ConfigError):
        EditorConfig(reference_note="")
    with pytest.raises(ConfigError):
        EditorConfig(validation={})
    with pytest.raises(ConfigError):
        EditorConfig(reference_status="approved")


@pytest.mark.parametrize("text", [
    "", "[]", "geometry: [", "geometry: 1\ngeometry: 2",
    "!!python/object/apply:builtins.eval ['1 + 1']", "1: 2",
])
def test_bad_yaml_rejected(tmp_path, text):
    path = tmp_path / "invalid.yaml"
    path.write_text(text, encoding="utf-8")
    with pytest.raises(ConfigError):
        load_config(path)


@pytest.mark.parametrize("mutation", ["unknown", "missing", "section_typo", "wrong_section"])
def test_strict_fields(tmp_path, mutation):
    data = EditorConfig().snapshot()
    if mutation == "unknown":
        data["extra"] = 1
    elif mutation == "missing":
        del data["validation"]
    elif mutation == "section_typo":
        data["geometry"]["paralell_threshold"] = 1e-5
    else:
        data["ui"] = []
    path = tmp_path / "config.yaml"
    path.write_text(yaml.safe_dump(data), encoding="utf-8")
    with pytest.raises(ConfigError):
        load_config(path)


def test_default_loading_does_not_depend_on_cwd(tmp_path, monkeypatch):
    monkeypatch.chdir(tmp_path)
    assert load_config() == EditorConfig()
    with pytest.raises(FileNotFoundError):
        load_config("missing.yaml")
